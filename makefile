all:
	sudo venv/bin/python main.py

video: 
	sudo venv/bin/python model.py --video

image:
	sudo venv/bin/python model.py --image

install: venv
	venv/bin/pip install --upgrade pip setuptools wheel
	venv/bin/pip install -r requirements.txt
	if [ "$(shell uname -s)" = "Darwin" ]; then brew install imagemagick; fi
	if [ "$(shell uname -s)" = "Linux" ]; then sudo apt-get install imagemagick; fi

venv:
	if [ "$(shell uname -s)" = "Darwin" ]; then python3.11 -m venv venv/; fi
	if [ "$(shell uname -s)" = "Linux" ]; then python3.7 -m venv venv/; fi