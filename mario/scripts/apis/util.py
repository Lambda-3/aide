import re

_first_cap_re = re.compile('(.)([A-Z][a-z]+)')
_all_cap_re = re.compile('([a-z0-9])([A-Z])')


def camel_case_to_underscore(name):
    s1 = _first_cap_re.sub(r'\1_\2', name)
    return _all_cap_re.sub(r'\1_\2', s1).lower()


def camel_case_to_space(name):
    s1 = _first_cap_re.sub(r'\1 \2', name)
    return _all_cap_re.sub(r'\1 \2', s1).lower()


def underscore_to_space(name):
    """

    :type name: str
    """
    return name.replace("_", " ")


def to_space(name):
    return camel_case_to_space(underscore_to_space(name))


def get_doc(something_with_doc):
    try:
        "\n".join(x.strip() for x in something_with_doc.__doc__.strip().splitlines())
    except:
        return ""
