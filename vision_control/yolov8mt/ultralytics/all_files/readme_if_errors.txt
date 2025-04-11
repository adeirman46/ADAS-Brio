/lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found (required by /usr/local/zed/lib/libsl_zed.so)

solution = conda install -c conda-forge gcc=12.1.0

ImportError: /lib/x86_64-linux-gnu/libgobject-2.0.so.0: undefined symbol:ffi_type_uint32, version LIBFFI_BASE_7.0

solution = export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libffi.so.7
