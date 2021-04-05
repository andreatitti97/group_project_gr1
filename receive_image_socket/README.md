# receive_image_socket
default:jpg (for png change the launchfile from `socketServerros2.py` to `socketServerros.py` )

the ip should be found automatically; if not so (gives error on the creation of the server ):    

- set the ip address in `./drone_publish_image/socketServerros2.py`, in `subserver((IP_ADDRESS, 8888))`

run `roslaunch drone_publish_image server.launch `