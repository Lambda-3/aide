_places = dict()


def tag_place(x, y, name):
    _places[name] = (x, y)


def get_tagged_coordinates(name):
    try:
        return _places[name]
    except KeyError:
        return None
