# Gemelo digital PyBullet del enjambre ACO
FROM python:3.11-slim

# g++ solo se usa si pip no encuentra wheel de pybullet (p. ej. Mac con chip M / ARM)
RUN apt-get update \
 && apt-get install -y --no-install-recommends build-essential \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY *.py ./

ENV PYTHONUNBUFFERED=1
# 8080 = visor web,  4210/udp = red del enjambre
EXPOSE 8080 4210/udp

CMD ["python", "simulacion.py", "--modo", "web", "--ap", "192.168.4.1"]
