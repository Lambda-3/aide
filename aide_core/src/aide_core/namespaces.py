import sys
import types


class Namespaces(types.ModuleType):
    """
    Fake module namespace class.

    When using ``from namespaces import foo`` you actually "import" from this class. That is, you call
    ``Namespaces().__getattr__('foo')``. This returns a `NamespaceWrapper` object.
    """

    __all__ = lambda x: x

    def __getattr__(self, item):
        """
        Returns a NamespaceWrapper object.

        When you import from namespaces, this method is called and returns a NamespaceWrapper object, which behaves
        like a usual python data object with some convenience methods.

        The ``NamespaceWrapper`` object can be used in two ways to get ``SubjectWrapper`` objects with the name `x`:

        * ``foo.x``
        * ``foo[x]``

        Furthermore ``foo._`` returns a ``SubjectWrapper`` object with a randomly chosen uuid.

        :param item: Name of the NamespaceWrapper object to get.
        :return: a NamespaceWrapper object.
        """
        # local imports because of some python import magic
        from rdflib import URIRef
        from uuid import uuid4 as uuid
        from aide_core.apis.rdf_utils import Triple
        from rdflib import Namespace, RDF
        import inspect
        properties = Namespace("http://lambda3.org/aide/properties/")
        classes = Namespace("http://lambda3.org/aide/classes/")

        namespace = item

        # local definition because of the same python import magic
        class SubjectWrapper:
            def __init__(self, name=None, ns=namespace):
                if not name:
                    self.__dict__["_subject"] = URIRef(
                        "http://lambda3.org/aide/{}/{}".format(namespace, "uuid-%s" % uuid()))
                else:
                    self.__dict__["_subject"] = URIRef("http://lambda3.org/aide/{}/{}".format(ns, name))

                # used for references and as a replacement of instanceof checks.
                self._is_subject = True

                # this is the file name of the importing extractor
                importee_fn = inspect.stack(0)[2][1].rsplit("/", 1)[-1].rsplit(".py", 1)[0].rsplit("_extractor", 1)[0]

                # default class. added to the message when no rdf class is defined explicitly.
                self._default_cls = "genClass_{}".format(importee_fn)

            def format_key_value(self, key, value):
                if key == "a":
                    key = RDF.type
                    value = classes[value]
                else:
                    key = properties[key]
                    value = value._subject if getattr(value, "_is_subject", None) else value
                return key, value

            def to_rdf_triples(self):
                if not getattr(self, "a", False):
                    self.a = self._default_cls
                triples = tuple(Triple(
                    self._subject,  # subject
                    *self.format_key_value(k, v)  # predicate and object
                ) for k, v in self.__dict__.items() if not k.startswith("_"))
                return triples

            def __getitem__(self, item):
                return self.__dict__[item]

            def __setattr__(self, key, value):
                self.__setitem__(key, value)

            def __setitem__(self, key, value):
                if key == "_subject":
                    raise AttributeError("'_subject' is a reserved attribute!")
                elif key == "type":
                    self.__dict__["a"] = value
                else:
                    self.__dict__[key] = value

            def __str__(self):
                return str(self.to_rdf_triples())

            def __repr__(self):
                return self.__str__()

        # This is the Wrapper definition
        NamespaceWrapper = type(
            "NamespaceWrapper",  # class name
            (object,),  # 'new style' class
            {  # class dict
                'uuid': lambda self: SubjectWrapper(),  # namespace.uuid() yields a random uuid subject
                '_': property(lambda self: SubjectWrapper()),  # namespace._ yields a random uuid subject
                '__getattr__': (lambda self, item: SubjectWrapper(item)),  # namespace.x yields a subject with name x
                '__getitem__': (lambda self, key: SubjectWrapper(key))  # namespace[x] yields a subject with name x
            }
        )
        return NamespaceWrapper()


# set this module to the class defined above. This is done so when importing from namespaces you actually "import" from
# the class
sys.modules[__name__] = Namespaces(name=__name__)
