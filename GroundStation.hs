{-# LANGUAGE OverloadedStrings, RecordWildCards #-}
import System.IO
import Control.Applicative
import Control.Concurrent hiding (yield)
import System.Random
import           Data.ByteString (ByteString)
import qualified Data.ByteString as BS
import qualified Data.ByteString.Lazy as BSL
import           Data.Binary.Put
import Numeric
import Pipes
import qualified Pipes.Prelude as P
import qualified Pipes.ByteString as B
import Xbee as X
import Linear
import Data.Attoparsec.ByteString as P
import Data.Bits
import Data.Word
import Data.Int
import Data.Maybe
import Control.Monad
import Control.Monad.Trans.Either

import System.Serial
import Options.Applicative as O
import Graphics.UI.GLFW as G

import Pipes.Concurrent as PC
import Graphics.Orientation

joystickPipe :: EitherT String IO (Producer [Double] IO ())
joystickPipe = do
    res <- lift G.init
    unless res $ left "Unable to initialise GLFW"

    lift $ setErrorCallback $ Just $ \err str -> putStrLn (show err) >> putStrLn str

    pres <- lift $ joystickPresent Joystick'1
    unless pres $ left "No joystick present"

    name <- lift $ getJoystickName Joystick'1
    name <- maybe (left "Cannot get joystick name") right name
    lift $ putStrLn $ "Found a: " ++ name

    let loop = do
        res <- lift $ getJoystickAxes Joystick'1
        case res of 
            Nothing -> return ()
            Just x  -> do
                yield x
                lift $ threadDelay 100000
                loop

    return loop

data ControlInputs = ControlInputs {
    throttle    :: Double,
    orientation :: Quaternion Double
} deriving (Show)

toQuat :: Monad m => Pipe [Double] ControlInputs m r
toQuat = P.map convert
    where
    convert axes = ControlInputs ((1 - axes !! 2) / 2) (axisAngle (V3 0 0 1) (- (axes !! 0)) * axisAngle (V3 1 0 0) (axes !! 1) * axisAngle (V3 0 1 0) (- (axes !! 3)))

data Options = Options {
    device :: Maybe String
}

data Packet = 
      XmitStat TXStatus
    | Receive  RecvPacket
    deriving (Show)

data Transmission = 
      Ack
    | Exception
    | Telemetry (Quaternion Double)
    deriving (Show)

data Reception = 
      Ping
    | Control ControlInputs
    | Gains   Double Double Double Double Double
    | Reset
    deriving (Show)

putDoubleAsFix16 :: Double -> Put
putDoubleAsFix16 x = putWord32le $ truncate $ x * 65536

serializeR :: Reception -> ByteString
serializeR x = BSL.toStrict $ runPut (serializeR' x)
    where
    serializeR' Ping                                                    = putWord8 0
    serializeR' (Control (ControlInputs thr (Quaternion w (V3 x y z)))) = putWord8 1 <* putDoubleAsFix16 thr <* putDoubleAsFix16 w <* putDoubleAsFix16 x <* putDoubleAsFix16 y <* putDoubleAsFix16 z
    serializeR' (Gains t w x y z)                                       = putWord8 2 <* putDoubleAsFix16 t   <* putDoubleAsFix16 w <* putDoubleAsFix16 x <* putDoubleAsFix16 y <* putDoubleAsFix16 z
    serializeR' Reset                                                   = putWord8 3

buildQuat w x y z = Quaternion w (V3 x y z)

anyWord32 :: P.Parser Word32
anyWord32 = func <$> anyWord8 <*> anyWord8 <*> anyWord8 <*> anyWord8
    where
    func w x y z = (fromIntegral z `shift` 24) .|. (fromIntegral y `shift` 16) .|. (fromIntegral x `shift` 8) .|. fromIntegral w

anyFix16 :: P.Parser Double
anyFix16 = ((/ 65536) . (fromIntegral :: (Int32 -> Double)) . fromIntegral) <$> anyWord32

processRecv :: RecvPacket -> Either String Transmission
processRecv RecvPacket{..} = parseOnly (ack <|> telemetry <|> exc) recvData
    where
    ack       = Ack       <$ word8 0
    telemetry = Telemetry <$ word8 1 <*> (buildQuat <$> anyFix16 <*> anyFix16 <*> anyFix16 <*> anyFix16)
    exc       = Exception <$ word8 2

doPrint tele oth = for cat $ \dat -> lift $ 
    case dat of
        XmitStat stat -> putStrLn $ "Transmittion status: " ++ show stat
        Receive  pkt  -> let res = processRecv pkt in 
            case res of
                Left err               -> return ()
                Right Ack              -> void $ atomically $ PC.send oth "ACK"
                Right (Telemetry quat) -> void $ atomically $ PC.send tele quat

doIt Options{..} = eitherT putStrLn return $ do
    --rnd <- randomIO
    --print $ map (flip showHex "") $ BS.unpack res

    (outputTelemetry, inputTelemetry) <- lift $ spawn unbounded
    (outputOther,     inputOther)     <- lift $ spawn unbounded

    h <- lift $ openSerial (fromMaybe "/dev/ttyUSB0" device) B38400 8 One NoParity NoFlowControl
    lift $ forkIO $ runEffect (B.fromHandle h >-> packetsPipe ((XmitStat <$> txStat) <|> (Receive <$> receive)) >-> doPrint outputTelemetry outputOther)

    lift $ forkIO $ runEffect $ fromInput inputOther     >-> P.print

    input <- lift $ readFile "suzanne.obj"
    let (verts, norms) = parseObj $ Prelude.lines input
    pipe <- join $ lift $ liftM hoistEither $ drawOrientation verts norms
    lift $ forkIO $ runEffect $ fromInput inputTelemetry >-> P.map (fmap realToFrac) >-> pipe

    let pipe = for cat $ \dat -> lift $ do 
        let res = BSL.toStrict $ runPut $ X.send (Just 0x5678) (Just 0x00) (Just 0x01) (serializeR $ Control dat) 
        BS.hPut h res

    jp <- joystickPipe 
    lift $ runEffect $ jp >-> toQuat >-> pipe
    
main = execParser opts >>= doIt
    where
    opts   = info (helper <*> parser) (fullDesc <> progDesc "Ground Station" <> O.header "Ground Station")
    parser = Options <$> optional (strOption (long "device" <> metavar "DEV"))

