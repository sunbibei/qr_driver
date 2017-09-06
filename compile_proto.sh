PROTO_SRC_DIR=config

# Hack to compile directly into src folders for now
CPP_OUT_DIR=include/system/platform/protocol/proto
mkdir -p "$CPP_OUT_DIR"

protoc -I=$PROTO_SRC_DIR --cpp_out=$CPP_OUT_DIR $PROTO_SRC_DIR/dragon.proto

echo "Done"
