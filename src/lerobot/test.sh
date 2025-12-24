# python xlerobot_manual_joint_control.py --port /dev/ttyACM1  --raw --joint left_arm_gripper --value 3500 --speed slow
# python xlerobot_manual_joint_control.py --speed slow --port /dev/ttyACM1

# python xlerobot_manual_joint_control.py --port /dev/ttyACM1  --raw --interactive 

# python xlerobot_manual_joint_control.py --port /dev/ttyACM1   --joint left_arm_gripper --value 0
#   left_arm_shoulder_pan          - Shoulder pan rotation (horizontal rotation)
#   left_arm_shoulder_lift         - Shoulder lift (up/down)
#   left_arm_elbow_flex            - Elbow flex (bend)
#   left_arm_wrist_flex            - Wrist flex (up/down)
#   left_arm_wrist_roll            - Wrist roll (rotation)
#   left_arm_gripper 
#   --port PORT      USB port for the robot (default: /dev/ttyACM0)
#   --use-right-arm  Use right arm instead of left arm
#   --raw            Use raw motor values (0-4095) instead of normalized values
#   --joint JOINT    Joint name to control (e.g., left_arm_shoulder_pan)
#   --value VALUE    Value to set (angle/velocity for normal mode, raw integer for raw mode)
#   --interactive    Enter interactive mode for multiple commands
#   --list           List all available joints
# python -m lerobot.robots.xlerobot.xlerobot_single_arm_host --robot.id=None
# python -m lerobot.4_xlerobot_single_arm_teleop_leader_client
python -m lerobot.record \
    --robot.type=xlerobot_single_arm_client \
    --robot.remote_ip=192.168.110.29 \
    --robot.id=my_xlerobot \
    --teleop.type=so101_leader \
    --teleop.port=COM3 \
    --teleop.id=my_leader_arm \
    --dataset.repo_id=your_username/xlerobot_dataset \
    --dataset.single_task="把橘子放到篮子里" \
    --dataset.num_episodes=10 \
    --display_data=true

python -m lerobot.record `
    --robot.type=xlerobot_single_arm_client `
    --robot.remote_ip=192.168.110.29 `
    --robot.id=my_xlerobot `
    --teleop.type=so101_leader `
    --teleop.port=COM3 `
    --teleop.id=my_leader_arm `
    --dataset.repo_id=wangranryan/xlerobot_lsd `
    --dataset.single_task="把螺丝刀放到篮子里" `
    --dataset.num_episodes=5 `
    --display_data=true `
    --play_sounds=false `
    --resume=true 

python -m lerobot.record `
    --robot.type=xlerobot_single_arm_client `
    --robot.remote_ip=192.168.110.29 `
    --robot.id=my_xlerobot `
    --teleop.type=so101_leader `
    --teleop.port=COM3 `
    --teleop.id=my_leader_arm `
    --dataset.repo_id=wangranryan/xlerobot_lsd `
    --dataset.single_task="把螺丝刀放到篮子里" `
    --dataset.num_episodes=10 `
    --display_data=true `
    --play_sounds=false 

lerobot-train `
  --policy.path=lerobot/smolvla_base `
  --policy.repo_id=wangranryan/xlerobot_lsd_1 `
  --dataset.repo_id=wangranryan/xlerobot_lsd `
  --batch_size=8 `
  --steps=20000 `
  --output_dir=outputs/train/my_smolvla `
  --job_name=my_smolvla_training `
  --policy.device=cuda `
  --wandb.enable=false


  huggingface-cli upload wangranryan/xlerobot_lsd "C:\Users\wr\.cache\huggingface\lerobot\wangranryan\xlerobot_lsd" --repo-type dataset




