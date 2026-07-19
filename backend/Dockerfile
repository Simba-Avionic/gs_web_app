FROM python:3.12

RUN apt-get update && apt-get install -y gcc
WORKDIR /app
COPY requirements.txt .

#ADD "https://www.random.org/cgi-bin/randbyte?nbytes=10&format=h" skipcache
RUN pip install --no-cache-dir -r requirements.txt

COPY . .
EXPOSE 2137
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "2137"]