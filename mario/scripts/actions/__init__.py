def __update():
    import os
    import config
    api_files = []
    for (dirpath, _, file_names) in os.walk(config.ACTION_PROVIDERS_PATH):
        # api_files.extend([dirpath + "/" + x for x in file_names if not x.startswith("__") and x.endswith(".py")])
        # return api_files
        for file_name in [f for f in file_names if not f.startswith("__") and f.endswith(".py")]:
            exec """import {0} as {0}""".format(file_name.rsplit(".", 1)[0])
