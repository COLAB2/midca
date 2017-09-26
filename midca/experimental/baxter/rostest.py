import subprocess
import select
import time

cmd = subprocess.Popen(['bash'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

poll = select.poll()
poll.register(cmd.stdout.fileno(),select.POLLIN)

# Write the first command
command = "cd ~/baxter_ws\n"
cmd.stdin.write(command)
cmd.stdin.flush() # Must include this to ensure data is passed to child process
time.sleep(1)
command = ". ~/baxter_ws/baxter.sh\n"
cmd.stdin.write(command)
cmd.stdin.flush() # Must include this to ensure data is passed to child process
ready = poll.poll(500)
if ready:
   result = cmd.stdout.readline()
   print result
time.sleep(2)
# Write the second command
command = "rosrun baxter_tools enable_robot.py -e\n"
cmd.stdin.write(command)
cmd.stdin.flush() # Must include this to ensure data is passed to child process
ready = poll.poll(500)
if ready:
   result = cmd.stdout.readline()
   print result
