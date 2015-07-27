{-# LANGUAGE RecordWildCards #-}
module FromQuad (
        FromQuad(..),
        processRecv
    ) where

import Control.Applicative
import Data.Bits
import Data.Word
import Data.Int

import Data.Attoparsec.ByteString as P
import Linear
import Xbee
        
-- Data received from the quadcopter
data FromQuad = 
      Ack
    | Exception
    | Telemetry (Quaternion Double)
    deriving (Show)

-- Parsing utilities
anyWord32 :: P.Parser Word32
anyWord32 = func <$> anyWord8 <*> anyWord8 <*> anyWord8 <*> anyWord8
    where
    func w x y z = (fromIntegral z `shift` 24) .|. (fromIntegral y `shift` 16) .|. (fromIntegral x `shift` 8) .|. fromIntegral w

anyFix16 :: P.Parser Double
anyFix16 = ((/ 65536) . (fromIntegral :: (Int32 -> Double)) . fromIntegral) <$> anyWord32

-- Parse a packet received from the quadcopter
processRecv :: RecvPacket -> Either String FromQuad
processRecv RecvPacket{..} = parseOnly (ack <|> telemetry <|> exc) recvData
    where
    ack       = Ack       <$ word8 0
    telemetry = Telemetry <$ word8 1 <*> (buildQuat <$> anyFix16 <*> anyFix16 <*> anyFix16 <*> anyFix16)
    exc       = Exception <$ word8 2
    buildQuat w x y z = Quaternion w (V3 x y z)

