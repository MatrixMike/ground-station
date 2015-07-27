module ToQuad (
        ControlInputs(..),
        GainSetting(..),
        ToQuad(..),
        serializeR
    ) where

import           Linear
import           Data.ByteString (ByteString)
import qualified Data.ByteString.Lazy as BSL
import           Data.Binary.Put

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
   
-- Serialization utilities
putDoubleAsFix16 :: Double -> Put
putDoubleAsFix16 x = putWord32le $ truncate $ x * 65536

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

