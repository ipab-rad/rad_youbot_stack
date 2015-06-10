from moveit_commander import MoveGroupCommander

if name == '__main__':
  group = MoveGroupCommander("arm")
  
  # move to a random target
  group.set_random_target()
  group.go()