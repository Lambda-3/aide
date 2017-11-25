def _update():
    """
    Function in order to import all modules in this namespace. (s.t. you can call apis.x without explicitly importing
    it)
    """
    import os
    import sys
    import importlib
    from aide_core import config
    api_files = []
    for (dirpath, _, file_names) in os.walk(config.APIS_PATH):
        for file_name in [f for f in file_names if not f.startswith("__") and f.endswith(".py")]:
            name = file_name.rsplit(".", 1)[0]
            mod = importlib.import_module("aide_core.apis.{}".format(name))
            setattr(sys.modules[__name__], name, mod)



