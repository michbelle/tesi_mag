# Tesi laurea magistrale

## Mini

basato su humble

## resto basato su jazzy

### git dowload

if the first time you need to launch

```bash
git submodule update --init --recursive
```

else

```bash
git submodule update --recursive
```


## build comand

```bash
colcon build --symlink-install
```

### devcontainer

start ssh agent
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/
```

then run
```bash
DOCKER_BUILDKIT=1 docker build -t test2 . --ssh default
```
