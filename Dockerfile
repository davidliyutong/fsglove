FROM davidliyutong/idekube-container:featured-base-v0.3.8

COPY . /opt/fsglove
WORKDIR /opt/fsglove

RUN . /opt/miniconda/etc/profile.d/conda.sh && conda create -n fsglove python=3.10
RUN . /opt/miniconda/etc/profile.d/conda.sh && conda activate fsglove && pip install --upgrade pip
RUN . /opt/miniconda/etc/profile.d/conda.sh && conda activate fsglove && pip install -r requirements.txt

RUN ln -s /lib/x86_64-linux-gnu/libdl.so.2 /lib/x86_64-linux-gnu/libdl.so
RUN apt-get update && apt-get install -y minizip && rm -rf /var/lib/apt/lists/*