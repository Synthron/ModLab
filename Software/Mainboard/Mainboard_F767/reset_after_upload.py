Import('env')

env.Prepend(
  UPLOADERFLAGS=["--reset"]
)