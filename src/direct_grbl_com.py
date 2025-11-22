from main import *
from grbl import *
from local.local_constants import *

grbl_comm = GrblCommunicator()
grbl_comm.serial_connect(do_ping=False)

# Do pre-programmed actions here

# Allow user to interact with GRBL
grbl_commands_list = [member.name for member in GrblCmd]
print(f"Send a move in the form 'G1 X__ Z__ F__' or a command from the list below.")
print(f"Available GRBL commands: {grbl_commands_list}")
while True:
    user_input = input(">>")
    if user_input in grbl_commands_list:
        x = getattr(GrblCmd, user_input).value
        print(f"Sending: {x}")
        grbl_comm.direct_write(x)
    else:
        x = bytes(f"{user_input}\n",  'utf-8')
        print(f"Sending: {x}")
        grbl_comm.direct_write(x)
    time.sleep(0.1)


