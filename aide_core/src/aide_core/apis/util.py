import re

from pylint import lint
from rospy import logwarn, loginfo
from pylint import epylint as lint

_first_cap_re = re.compile('(.)([A-Z][a-z]+)')
_all_cap_re = re.compile('([a-z0-9])([A-Z])')


def camel_case_to_underscore(name):
    s1 = _first_cap_re.sub(r'\1_\2', name)
    return _all_cap_re.sub(r'\1_\2', s1).lower()


def camel_case_to_space(name):
    s1 = _first_cap_re.sub(r'\1 \2', name)
    return _all_cap_re.sub(r'\1 \2', s1).lower()


def underscore_to_camel_case(name):
    return "".join(
        map(lambda x: x.capitalize(), underscore_to_space(name).split(" ")))


def underscore_to_space(name):
    """

    :type name: str
    """
    return name.replace("_", " ")


def to_space(name):
    return camel_case_to_space(underscore_to_space(name))


def get_doc(something_with_doc, strip_annotations=True):
    try:
        doc_lines = (x.strip() for x in something_with_doc.__doc__.strip().splitlines())
        if strip_annotations:
            return "\n".join(line for line in doc_lines if not line.startswith(":"))
        else:
            return "\n".join(doc_lines)
    except:
        loginfo(
            "Getting doc failed for function {}".format(something_with_doc))
        return ""


def get_type_hints(func):
    args = dict()
    errors = []
    doc = get_doc(func, strip_annotations=False)
    for line in doc.splitlines():
        if line.startswith(":type"):
            try:
                name, type = line.split("type", 1)[1].split(":", 1)
                args[name.strip()] = type.strip()
            except (ValueError, IndexError):
                error_msg = "Malformatted parameter type hinting: {} ".format(
                    line)
                logwarn(error_msg)
                errors.append(error_msg)
        elif line.startswith(":rtype"):
            try:
                type = line.split("rtype:", 1)[1]
                args["return"] = type.strip()
            except (ValueError, IndexError):
                error_msg = "Malformatted return type hinting: {}".format(line)
                logwarn(error_msg)
                errors.append(error_msg)
    return args, errors


def apply_lint(path):
    options = """%s --msg-template='{path}({line:3d}:{column:2d}): [{obj}] {msg}' 
                    --disable=C,R,I""" % path
    out, err = lint.py_run(options, return_std=True)
    return "".join(err.readlines()) + "".join(out.readlines())
