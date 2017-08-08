def __update():
    import os
    import sys
    import importlib
    from aide_core import config
    api_files = []
    for (dirpath, _, file_names) in os.walk(config.APIS_PATH):
        # api_files.extend([dirpath + "/" + x for x in file_names if not x.startswith("__") and x.endswith(".py")])
        # return api_files
        for file_name in [f for f in file_names if not f.startswith("__") and f.endswith(".py")]:
            name = file_name.rsplit(".", 1)[0]
            # print(name)

            # exec ("""import aide_core.apis.{0} as {0}""".format(name))
            mod = importlib.import_module("aide_core.apis.{}".format(name))
            setattr(sys.modules[__name__], name, mod)
            # print(getattr(sys.modules[__name__], name))


# __update()
# # print("="*20)
# # print eval("joke")
# # asdf
