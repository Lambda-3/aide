def _update():
    """
    Function in order to import all modules in this namespace. (s.t. you can call apis.x without explicitly importing
    it)
    """
    import os
    from aide_core import config
    api_files = []
    for (dirpath, _, file_names) in os.walk(config.ACTION_PROVIDERS_PATH):

        for file_name in [f for f in file_names if not f.startswith("__") and f.endswith(".py")]:
            exec("""import actions.{0} as {0}""".format(file_name.rsplit(".", 1)[0]))
