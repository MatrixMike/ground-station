{-# LANGUAGE RecordWildCards, GADTs #-}
module Joystick (
        joystickPipe,
        processJoystick
    ) where

import Control.Monad.Trans.Either

import           Pipes
import qualified Pipes.Prelude    as P
import           Graphics.UI.GLFW as G
import           Linear
import           Control.Monad.Trans.Maybe
import           Control.Monad
import           Control.Concurrent hiding (yield)

import ToQuad
import Config

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

