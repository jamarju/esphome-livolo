# ESPHome + Livolo switch integration

Integrates with 2-way (1-gang or 2-gang) Livolo switches via the COM line by simulating being another Livolo switch. Wiring diagram available upon request.

## Pre-reqs and base skeleton

```
apt install virtualenvwrapper
mkvirtualenv -p /usr/bin/python2.7 esphome
workon esphome
pip install esphome
pip install --upgrade voluptuous==0.11.5
```

Base skeleton was created with:

```
esphome livolo.yaml wizard
```

## Compile

The export is important before any other compile relate command, otherwise the compiler fails with internal error.

```
export LC_ALL=C
esphome livolo.yaml compile
```

## Run and serial debug

```
esphome livolo.yaml run
```

