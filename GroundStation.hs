{-# LANGUAGE OverloadedStrings, RecordWildCards #-}
import System.IO
import Control.Applicative
import Control.Concurrent
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

import System.Serial
import Options.Applicative as O

import Pipes.Concurrent as PC
import Graphics.Orientation

data Options = Options {
    device :: Maybe String
}

data Packet = 
      XmitStat TXStatus
    | Receive  RecvPacket
    deriving (Show)

data Transmission = 
      Ack
    | Telemetry (Quaternion Double)
    deriving (Show)

buildQuat w x y z = Quaternion w (V3 x y z)

anyWord32 :: P.Parser Word32
anyWord32 = func <$> anyWord8 <*> anyWord8 <*> anyWord8 <*> anyWord8
    where
    func w x y z = (fromIntegral z `shift` 24) .|. (fromIntegral y `shift` 16) .|. (fromIntegral x `shift` 8) .|. fromIntegral w

anyFix16 :: P.Parser Double
anyFix16 = ((/ 65536) . (fromIntegral :: (Int32 -> Double)) . fromIntegral) <$> anyWord32

processRecv :: RecvPacket -> Either String Transmission
processRecv RecvPacket{..} = parseOnly (ack <|> telemetry) recvData
    where
    ack       = Ack       <$ word8 0
    telemetry = Telemetry <$ word8 1 <*> (buildQuat <$> anyFix16 <*> anyFix16 <*> anyFix16 <*> anyFix16)

doPrint tele oth = for cat $ \dat -> lift $ 
    case dat of
        XmitStat stat -> putStrLn $ "Transmittion status: " ++ show stat
        Receive  pkt  -> let res = processRecv pkt in 
            case res of
                Left err               -> return ()
                Right Ack              -> void $ atomically $ PC.send oth "ACK"
                Right (Telemetry quat) -> void $ atomically $ PC.send tele quat

doIt Options{..} = do
    --rnd <- randomIO
    --print $ map (flip showHex "") $ BS.unpack res

    (outputTelemetry, inputTelemetry) <- spawn unbounded
    (outputOther,     inputOther)     <- spawn unbounded

    h <- openSerial (fromMaybe "/dev/ttyUSB0" device) B38400 8 One NoParity NoFlowControl
    forkIO $ runEffect (B.fromHandle h >-> packetsPipe ((XmitStat <$> txStat) <|> (Receive <$> receive)) >-> doPrint outputTelemetry outputOther)

    forkIO $ runEffect $ fromInput inputOther     >-> P.print

    input <- readFile "suzanne.obj"
    let (verts, norms) = parseObj $ Prelude.lines input
    Right pipe <- drawOrientation verts norms
    forkIO $ runEffect $ fromInput inputTelemetry >-> P.map (fmap realToFrac) >-> pipe

    let pipe = for cat $ \dat -> lift $ do 
        let res = BSL.toStrict $ runPut $ X.send (Just 0x5678) (Just 0x00) (Just 0x01) (BS.pack (map (toEnum . fromEnum) dat))
        putStrLn $ "Transmitting: " ++ dat
        BS.hPut h res

    runEffect $ P.stdinLn >-> pipe
    
main = execParser opts >>= doIt
    where
    opts   = info (helper <*> parser) (fullDesc <> progDesc "Ground Station" <> O.header "Ground Station")
    parser = Options <$> optional (strOption (long "device" <> metavar "DEV"))

