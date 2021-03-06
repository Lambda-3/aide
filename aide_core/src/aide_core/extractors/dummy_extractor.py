import random


from aide_core.extractors import AbstractPeriodicExtractor
from aide_core.namespaces import robot
from aide_core.apis import simple


class DummyExtractor(AbstractPeriodicExtractor):
    queue_size = 43

    @property
    def rate(self):
        return 0.2

    def extract(self):
        subj = robot.self
        subj.random = random.randint(0, 100)
        subj.random2 = simple.fancify_string("SDF")
        return subj
