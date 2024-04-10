#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EditingState {
    Create,
    Update,
    Delete,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RobotPart {
    Capsule,
    Joint,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JointCreationState {
    SelectFirstCapsule,
    SelectSecondCapsule,
    PlaceJoint,
}