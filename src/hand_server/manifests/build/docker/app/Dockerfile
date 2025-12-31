FROM golang:1.19.1 as builder

WORKDIR /opt/build
COPY . .

ENV GOPROXY https://goproxy.cn


RUN go mod vendor
RUN go build -o rfmocap-app ./cmd/app/app.go

FROM ubuntu:20.04

WORKDIR /opt/app

COPY --from=builder /opt/build/rfmocap-app ./rfmocap-app
COPY manifests/entrypoint/entrypoint.sh ./entrypoint.sh

RUN chmod +x ./entrypoint.sh

VOLUME /config
EXPOSE 18890

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["./entrypoint.sh"]