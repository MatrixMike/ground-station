{-# LANGUAGE OverloadedStrings, RecordWildCards, GADTs, TemplateHaskell, ScopedTypeVariables #-}
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
import Data.Either.Combinators
import Control.Monad.Trans.Maybe

import System.Serial
import Options.Applicative as O
import Graphics.UI.GLFW as G
import Data.Yaml
import Data.Aeson.TH hiding (Options)

import Graphics.Orientation

-- Configuration file
data Axes = Axes {
    throttleA :: Int,
    pitch     :: Int,
    roll      :: Int,
    yaw       :: Int
} deriving (Show)
deriveJSON defaultOptions ''Axes

data Trim = Trim {
    pitchT   :: Int,
    rollT    :: Int,
    pitchInc :: Double,
    rollInc  :: Double
} deriving (Show)
deriveJSON defaultOptions ''Trim

data Buttons = Buttons {
    reset  :: Int,
    arm    :: Int,
    disarm :: Int
} deriving (Show)
deriveJSON defaultOptions ''Buttons

data Gain = Gain {
    inc :: Int,
    dec :: Int,
    step :: Double
} deriving (Show)
deriveJSON defaultOptions ''Gain

data Config = Config {
    axes      :: Axes,
    yawStep   :: Double,
    trim      :: Trim,
    buttons   :: Buttons,
    horizGain :: Gain,
    yawGain   :: Gain
} deriving (Show)
deriveJSON defaultOptions ''Config

-- Command line arguments
data Options = Options {
    device      :: String,
    configFile  :: String,
    object      :: String,
    xbeeAddress :: Word16
}

-- Serialization utilities
putDoubleAsFix16 :: Double -> Put
putDoubleAsFix16 x = putWord32le $ truncate $ x * 65536

-- Parsing utilities
anyWord32 :: P.Parser Word32
anyWord32 = func <$> anyWord8 <*> anyWord8 <*> anyWord8 <*> anyWord8
    where
    func w x y z = (fromIntegral z `shift` 24) .|. (fromIntegral y `shift` 16) .|. (fromIntegral x `shift` 8) .|. fromIntegral w

anyFix16 :: P.Parser Double
anyFix16 = ((/ 65536) . (fromIntegral :: (Int32 -> Double)) . fromIntegral) <$> anyWord32

-- Pipe utilities
fork :: Monad m => Producer a m r -> Producer a (Producer a m) r
fork prod = runEffect $ hoist (lift . lift) prod >-> fork' 
    where 
    fork' = forever $ do
        res <- await
        lift $ yield res
        lift $ lift $ yield res

combine :: Monad m => Consumer a m r -> Consumer a m r -> Consumer a m r
combine x y = runEffect $ runEffect (fork func >-> hoist (lift . lift) x) >-> hoist lift y
    where
    func :: Monad m => Producer a (Consumer a m) r
    func = forever $ lift await >>= yield

-- Pipe to get joystick state
joystickPipe :: EitherT String IO (Producer ([Double], [JoystickButtonState]) IO ())
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
            x <- MaybeT $ lift $ getJoystickAxes    Joystick'1
            b <- MaybeT $ lift $ getJoystickButtons Joystick'1
            lift $ do
                yield (x, b)
                lift $ threadDelay 100000
            loop

    return $ const () <$> runMaybeT loop

-- Data types representing a packet to be sent to the quadcopter
data ControlInputs = ControlInputs {
    throttle    :: Double,
    orientation :: Quaternion Double
} deriving (Show)

data GainSetting = GainSetting {
    throttleP :: Double,
    horizP    :: Double,
    yawP      :: Double,
    horizD    :: Double,
    yawD      :: Double
} deriving (Show)

data ToQuad = 
      Ping
    | Control ControlInputs
    | Gains   GainSetting
    | Reset
    | Arm
    | Disarm
    deriving (Show)

-- Processing of Joystick input into quadcopter commands
data JoyState = JoyState {
    yawAccum   :: Double,
    pitchTrim  :: Double,
    rollTrim   :: Double,
    horizGainS :: Double,
    yawGainS   :: Double
} deriving (Show)

processJoystick :: (Monad m, m ~ IO) => Config -> Pipe ([Double], [JoystickButtonState]) ToQuad m r
processJoystick (Config Axes{..} yawStep Trim{..} Buttons{..} horizConfig yawConfig) = convert (JoyState 0 0 0 16384 16384)
    where
    convert js = do
        axes <- await
        fun axes
        where
        fun (axes, but) = do
            lift $ print js
            let ns = JoyState (yawAccum js   + yawStep  * yawVal) 
                              (pitchTrim js  + pitchInc * povPitch) 
                              (rollTrim js   + rollInc  * povRoll) 
                              (horizGainS js + upd gainUp gainDown (step horizConfig)) 
                              (yawGainS js   + upd yawGainUp yawGainDown (step yawConfig))
            yield $ Control $ ControlInputs throttleVal (axisAngle (V3 0 0 1) (yawAccum js) * axisAngle (V3 1 0 0) (rollVal + rollTrim ns) * axisAngle (V3 0 1 0) (pitchVal + pitchTrim ns))
            when (gainUp || gainDown || yawGainUp || yawGainDown) $ yield (Gains $ GainSetting 16384 (horizGainS ns) (yawGainS ns) 0 0)
            when resetPressed  $ yield Reset
            when armPressed    $ yield Arm 
            when disarmPressed $ yield Disarm
            convert ns
            where
            throttleVal   = (1 - axes !! throttleA) / 2
            pitchVal      = - (axes !! pitch)
            rollVal       = axes !! roll
            yawVal        = axes !! yaw
            povPitch      = - (axes !! pitchT)
            povRoll       = axes !! rollT
            gainUp        = but  !! (inc horizConfig) == JoystickButtonState'Pressed
            gainDown      = but  !! (dec horizConfig) == JoystickButtonState'Pressed
            yawGainUp     = but  !! (inc yawConfig)   == JoystickButtonState'Pressed
            yawGainDown   = but  !! (dec yawConfig)   == JoystickButtonState'Pressed
            resetPressed  = but  !! reset             == JoystickButtonState'Pressed
            armPressed    = but  !! arm               == JoystickButtonState'Pressed
            disarmPressed = but  !! disarm            == JoystickButtonState'Pressed
            upd x y inc   = if x then inc else if y then (-inc) else 0

-- Packet types received from the XBee
data FromXbee = 
      XmitStat TXStatus   --Status of the last transmission
    | Receive  RecvPacket --A packet received from the quadcopter
    deriving (Show)

-- Data received from the quadcopter
data FromQuad = 
      Ack
    | Exception
    | Telemetry (Quaternion Double)
    deriving (Show)

-- Pipe that only passes on joystick orientation commands
getOrientations :: Monad m => Pipe ToQuad (Quaternion Double) m ()
getOrientations = for cat func
    where
    func (Control (ControlInputs {..})) = yield orientation
    func _                              = return ()

-- Serialize a packet to be sent to the quadcopter
serializeR :: ToQuad -> ByteString
serializeR x = BSL.toStrict $ runPut (serializeR' x)
    where
    serializeR' Ping                                                    = putWord8 0
    serializeR' (Control (ControlInputs thr (Quaternion w (V3 x y z)))) = putWord8 1 <* putDoubleAsFix16 thr <* putDoubleAsFix16 w <* putDoubleAsFix16 x <* putDoubleAsFix16 y <* putDoubleAsFix16 z
    serializeR' (Gains (GainSetting t w x y z))                         = putWord8 2 <* putDoubleAsFix16 t   <* putDoubleAsFix16 w <* putDoubleAsFix16 x <* putDoubleAsFix16 y <* putDoubleAsFix16 z
    serializeR' Reset                                                   = putWord8 3
    serializeR' Arm                                                     = putWord8 4
    serializeR' Disarm                                                  = putWord8 5

-- Parse a packet received from the quadcopter
processRecv :: RecvPacket -> Either String FromQuad
processRecv RecvPacket{..} = parseOnly (ack <|> telemetry <|> exc) recvData
    where
    ack       = Ack       <$ word8 0
    telemetry = Telemetry <$ word8 1 <*> (buildQuat <$> anyFix16 <*> anyFix16 <*> anyFix16 <*> anyFix16)
    exc       = Exception <$ word8 2
    buildQuat w x y z = Quaternion w (V3 x y z)

-- Process a packet received from the quadcopter
forkFromXbee :: Monad m => Consumer FromXbee (Producer TXStatus (Producer RecvPacket m)) a
forkFromXbee = for cat $ \dat -> lift $ 
    case dat of
        XmitStat stat -> yield stat
        Receive  pkt  -> lift $ yield pkt

processFromQuad :: Pipe (Either String FromQuad) (Quaternion Double) IO a
processFromQuad = for cat $ \dat -> 
    case dat of
        Left err               -> lift $ putStrLn "Error parsing packet from quadcopter"
        Right Ack              -> lift $ putStrLn "Acknowledgement from quadcopter"
        Right (Telemetry quat) -> void $ yield quat

doIt Options{..} = eitherT putStrLn return $ do
    --Setup
    (config :: Config) <- join $ lift $ liftM hoistEither $ liftM (mapLeft show) $ decodeFileEither configFile
    lift $ print config
    lift $ setupGLFW
    h <- lift $ openSerial device B38400 8 One NoParity NoFlowControl

    --Create the orientation drawer
    input <- lift $ readFile object
    let (verts, norms) = parseObj $ Prelude.lines input
    pipe <- join $ lift $ liftM hoistEither $ drawOrientation verts norms

    --Process the data from the quadcopter
    lift $ forkIO $ 
        runEffect ( 
            runEffect (
                (runEffect (B.fromHandle h >-> packetsPipe ((XmitStat <$> txStat) <|> (Receive <$> receive)) >-> forkFromXbee)) 
                >-> P.print
            ) 
            >-> P.map processRecv >-> processFromQuad >-> P.map (fmap realToFrac) >-> pipe
        )

    --Process the data from the joystick to the quadcopter
    let serializeToQuad dat = BSL.toStrict $ runPut $ X.send (Just xbeeAddress) (Just 0x00) (Just 0x01) (serializeR dat)

    --joyDrawPipe <- join $ lift $ liftM hoistEither $ drawOrientation verts norms
    jp <- joystickPipe 
    lift $ runEffect $ jp >-> processJoystick config >-> P.map serializeToQuad >-> B.toHandle h --combine pipe (getOrientations >-> P.map (fmap realToFrac) >-> joyDrawPipe)
    
main = execParser opts >>= doIt
    where
    opts   = info (helper <*> parser) (fullDesc <> progDesc "Ground Station" <> O.header "Ground Station")
    parser = Options <$> (fromMaybe "/dev/ttyUSB0" <$> optional (strOption      (long "device"  <> short 'd' <> metavar "FILE" <> value "/dec/ttyUSB0"  <> showDefault <> help "Device file for serial input")))
                     <*> (fromMaybe "config.yaml"  <$> optional (strOption      (long "config"  <> short 'c' <> metavar "FILE" <> value "./config.yaml" <> showDefault <> help "YAML config file location")))
                     <*> (fromMaybe "suzanne.obj"  <$> optional (strOption      (long "object"  <> short 'o' <> metavar "FILE" <> value "suzanne.obj"   <> showDefault <> help "Obj file for oriented object to draw")))
                     <*> (fromMaybe 0x5678         <$> optional (O.option  auto (long "address" <> short 'a' <> metavar "NUM"  <> value 0x5678          <> showDefault <> help "Address of quadcopter xbee")))

