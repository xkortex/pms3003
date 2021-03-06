#!/usr/bin/env bash

# define flask app directory
flaskdir="/opt/pms3003/"

# install nginx and git
yum install nginx, git -y
amazon-linux-extras install nginx1.12 -y

# get pip
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python get-pip.py
rm -f get-pip.py

# install virtualenv for isolated Python environment
pip install virtualenv

# create flask virtual environment
mkdir $flaskdir && cd $flaskdir
virtualenv flask

# get code from github
git clone https://github.com/sylwesterf/pms3003.git

# copy viz files
cp pms3003/viz/py/{fun.py,requirements.txt,vizflask.py,wsgi.py} .
cp -a pms3003/viz/py/assets .

# activate script flask venv, install flask app requirements and run wsgi server
/bin/bash -c ". /opt/pms3003/flask/bin/activate; pip install -r requirements.txt; gunicorn --bind 0.0.0.0:80 wsgi:server &"
