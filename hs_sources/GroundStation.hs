{-# LANGUAGE OverloadedStrings, RecordWildCards, GADTs, TemplateHaskell, ScopedTypeVariables #-}
import           System.IO
import           Control.Concurrent           hiding (yield)
import           Data.ByteString (ByteString)
import qualified Data.ByteString.Lazy         as BSL
import qualified Pipes.ByteString             as B
import Linear
import Data.Word
import Data.Maybe
import Control.Monad
import Control.Monad.Trans.Either
import Data.Either.Combinators

import Pipes
import qualified Pipes.Prelude as P
import Data.Binary.Put
import System.Serial
import Options.Applicative as O
import Graphics.UI.GLFW as G
import Data.Yaml

import Graphics.Orientation
import Xbee as X

import ToQuad
import FromQuad
import Config
import Joystick

-- Command line arguments
data Options = Options {
    device      :: String,
    configFile  :: String,
    object      :: String,
    xbeeAddress :: Word16
}

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

-- Packet types received from the XBee
data FromXbee = 
      XmitStat TXStatus   --Status of the last transmission
    | Receive  RecvPacket --A packet received from the quadcopter
    deriving (Show)

-- Pipe that only passes on joystick orientation commands
getOrientations :: Monad m => Pipe ToQuad (Quaternion Double) m ()
getOrientations = for cat func
    where
    func (Control (ControlInputs {..})) = yield orientation
    func _                              = return ()

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
    parser = Options <$> (fromMaybe "/dev/ttyUSB0" <$> optional (strOption      (long "device"  <> short 'd' <> metavar "FILE" <> value "/dev/ttyUSB0"  <> showDefault <> help "Device file for serial input")))
                     <*> (fromMaybe "config.yaml"  <$> optional (strOption      (long "config"  <> short 'c' <> metavar "FILE" <> value "./config.yaml" <> showDefault <> help "YAML config file location")))
                     <*> (fromMaybe "suzanne.obj"  <$> optional (strOption      (long "object"  <> short 'o' <> metavar "FILE" <> value "suzanne.obj"   <> showDefault <> help "Obj file for oriented object to draw")))
                     <*> (fromMaybe 0x5678         <$> optional (O.option  auto (long "address" <> short 'a' <> metavar "NUM"  <> value 0x5678          <> showDefault <> help "Address of quadcopter xbee")))

