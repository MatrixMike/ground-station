{-# LANGUAGE TemplateHaskell #-}
module Config (
        Axes(..),
        Trim(..),
        Buttons(..),
        Gain(..),
        Config(..)
    ) where

import Data.Aeson.TH hiding (Options)

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

