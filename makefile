all:
	sudo venv/bin/python main.py

video: 
	sudo venv/bin/python video.py --video

install: venv
	venv/bin/pip install --upgrade pip setuptools wheel
	venv/bin/pip install -r requirements.txt

venv:
	if [ "$(shell uname -s)" = "Darwin" ]; then python3.11 -m venv venv/; fi
	if [ "$(shell uname -s)" = "Linux" ]; then python3.7 -m venv venv/; fi