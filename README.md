# controls
Robot control code for the manipulator and drivebase

# Joint States Should Use The Following Names:
`linear_rail`  
`shoulder`  
`elbow`  
`wrist_pitch`  
`wrist_roll`  


## To Do

### Drivetrain
- [x] auto stop all motors when controls signal is lost
- [x] differential drive
- [ ] tank drive
- [ ] torque vectoring control 
### Manipulator
- [x] auto stop all motors when controls signal is lost
- [x] single motor control
- [ ] inverse kinematics control
### Misc
- [x] can start/stop bash script
- [x] gui-independent voltage and current graphing
- [ ] integrated switching between can/uart
- [ ] wanderer2 backwards compatability
