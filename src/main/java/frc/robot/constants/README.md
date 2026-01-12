# Constants

All non-unitless measurements are in metric units: radians, kilograms, meters, seconds, etc. and combinations thereof.
Angles are always measured in radians.

We're not using WPI's UoM types here because it's not completely intuitive and
completely limited by Java not having useful features to make it good.

Reduction factors are always in the form of `output/input` from motor to drum (e.g. 10:1 gear ratio is 0.1).
The `ModuleConstants.DriveMotor.kReduction` was flipped on purpose - in rev's documentation, it's `input/output`.