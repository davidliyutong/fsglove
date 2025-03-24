AUTHOR = davidliyutong

.PHONY: task.pb.python_client
task.pb.python:
	@echo "===========> Generating protobuf files for python"
	@mkdir -p client/python
	@python -m grpc_tools.protoc -I./assets/pb \
	    --python_out=./src/api --grpc_python_out=./src/api ./assets/pb/imu_packet.proto