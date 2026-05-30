# stargazer — URDF → GLB converter image
# Build: docker build -t stargazer-converter .
# Run:   docker run --rm -v $(pwd):/work stargazer-converter models/go1/go1.urdf models/go1/go1.glb
FROM python:3.11-slim

RUN pip install --no-cache-dir trimesh pycollada numpy

COPY urdf2glb.py /converter/urdf2glb.py
WORKDIR /work
ENTRYPOINT ["python3", "/converter/urdf2glb.py"]
