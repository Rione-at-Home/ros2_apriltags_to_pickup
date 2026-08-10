"""
Predefined joint poses for the Crane+ arm.

All joint values are stored in DEGREES.

Joint order:
1. Base
2. Shoulder
3. Elbow
4. Wrist
5. Gripper
"""

HOME = [
    0,
    0,
    0,
    0,
    0,       

]

REST = [
    92,
    108,
    114,
    120,
    65
]
PRE_PICK = [
    0,
    35,
    -100,
    0,
    -35,
]

APPROACH = [
    0,
    90,
    -100,
    70,
    -35,
]

GRAB = [
    #0,
    #100,
    #-43,
    #45,
    #-35,
    0,
    100,
    -60,
    50,
    -10

]

GRAB2 = [
    0,
    97,
    -60,
    70,
    35,
]

LIFT = [
    #0,
    #10,
    #-45,
    #20,
    #35,
    0,
    -10,
    -60,
    50,
    -10
]

LEFT_PLACE = [
    90,
    95,
    -40,
    50,
    35,
]

RIGHT_PLACE = [
    -90,
    95,
    -40,
    50,
    -35,
]

LEFT_RELEASE = [
    90,
    95,
    -45,
    50,
    -25,
]

RIGHT_RELEASE = [
    -90,
    95,
    -45,
    50,
    -25,
]

THROW_READY = [
    0,
    90,
    -60,
    70,
    -35,
]

THROW = [
    0,
    5,
    -5,
    5,
    -35,
]

THROW_RELEASE = [
    0,
    -20,
    20,
    -20,
    -60,
]

CUSTOM1 = [
    0,
    134,
    -53,
    54,
    -58
]

CUSTOMGRAB = [
    0,
    134,
    -53,
    54,
    135
]

