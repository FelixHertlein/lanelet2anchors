FROM python:3.9-slim

RUN apt-get -y update
RUN apt-get install -y git cmake gcc
RUN apt-get install -y ffmpeg libsm6 libxext6  