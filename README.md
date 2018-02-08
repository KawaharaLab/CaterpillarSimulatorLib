# CaterpillarSimulatorLib
3D caterpillar simulator for Python implemented in Rust

## Install

```shell:
git clone https://github.com/KawaharaLab/CaterpillarSimulatorLib.git
cd CaterpillarSimulatorLib
python ./setup.py develop
```

or, if you are using pyenv, 

```shell:
pyenv exec python setup.py develop
```

Maybe sudo is required.


To use the caterpillar simulator in python,
```python
import caterpillar_lib import caterpillar
c = caterpillar.Caterpillar(5, (1,2,3), {})
```

## Quick run
To try this simulator out, simply run

```shell
python scripts/example.py
```

Simulation result is dump in "test_render.json".
You can render and check the result by,

```shell
blender --python scripts/render.py test_render.json
```
