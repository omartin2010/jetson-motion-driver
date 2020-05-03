# FROM omartin2010/robo-ev3-buster
FROM ev3dev/ev3dev-buster-rpi2-generic

RUN apt-get update && \
    apt-get install -y python3-pip

RUN pip3 install numpy \
    modern_robotics \
    python-ev3dev2 \
    paho-mqtt \
    git+https://github.com/microsoft/debugpy.git

ENV DEBIAN_FRONTEND="noninteractive"
RUN apt install -y tzdata
RUN ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime
RUN dpkg-reconfigure --frontend noninteractive tzdata

WORKDIR /app
COPY robot-def-pi.json /app
COPY src/*.py /app/
CMD [ "/bin/sh", "-c", "python3 startup.py -f robot-def-pi.json"]