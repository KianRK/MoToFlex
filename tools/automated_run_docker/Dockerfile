#Download base image ubuntu 20.04
FROM --platform=linux/amd64 nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04
#FROM --platform=linux/amd64 nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.0

# LABEL about the NaoDevils image
LABEL maintainer="oliver.urbann@tu-dortmund.de"
LABEL version="1.0"
LABEL description=""
ENV TZ=Asia/Dubai
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.0"
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt update && apt install -y gpg wget apt-utils software-properties-common 

RUN apt update && apt install -y --no-install-recommends sudo python3-pip git python3-dev python-is-python3 libglu1-mesa-dev freeglut3-dev mesa-common-dev autoconf autotools-dev libtool python3.11-dev autotools-dev automake build-essential nano
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1

RUN pip3 install jupyter pyzbar tqdm matplotlib ipython pip install tqdm gymnasium[classic-control] ipywidgets matplotlib pyvirtualdisplay stable_baselines3 wandb moviepy tensorboard
RUN pip3 install scipy

COPY run.sh /
RUN chmod +x /run.sh

# Run your experiment or execute the code
CMD ["/run.sh"]