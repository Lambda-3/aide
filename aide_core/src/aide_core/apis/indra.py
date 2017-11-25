import json
import requests

from aide_core.apis.util import to_space

DATA = {'corpus': 'wiki-2014',
         'model': 'W2V',
         'language': 'EN',
         'scoreFunction': 'COSINE'}

HEADERS = {
    'content-type': "application/json"
}

ENDPOINT = "http://localhost:8916/relatedness"


def get_semantic_relatedness(pairs):
    """
    Calls the indra endpoint to obtain pair-wise relatedness for a list of pairs.
    :param pairs: list of word pairs where each element looks like {"t1": word1, "t2": word2}
    :return: list of word pairs where each element looks like {"t1": word1, "t2": word2, "score": value}
    """
    DATA['pairs'] = pairs
    response = requests.request("POST", ENDPOINT, data=json.dumps(DATA), headers=HEADERS)

    response.raise_for_status()

    response = response.json()['pairs']
    return response


def sort_by_relatedness_with_id(word, words_with_ids, id_name="_id", format="{name}"):
    """
    Sorts a list of dictionaries by their relatedness to a given input.

    Which fields of a dictionary to use for comparison can be defined via the keyword argument 'format' using
    python's new style formatting, which field to take as id via 'id_name'.

    :param word: Plain text to compare the list to.
    :type word: str
    :param words_with_ids: List of dictionaries to sort.
    :type words_with_ids: list
    :param id_name: Key name of a single dict entry. Defaults to "_id".
    :type id_name: str
    :param format: String format template which defines which dict entries and in which order to use for comparison. Defaults to "_name"
    :type format: str
    :return: List of entries sorted by their semantic relatedness to word.
    :rtype: list
    """
    pairs = [{"t1": to_space(word),
              "t2": to_space(("{id} | " + format).format(
                  id=words_dict[id_name], **{k: v for k, v in words_dict.items() if not k == id_name}))}
             for words_dict in words_with_ids]
    result = sorted(get_semantic_relatedness(pairs), reverse=True, key=lambda x: x['score'])
    return [entry["t2"].split(" | ", 1)[0] for entry in result]


def sort_by_relatedness(word, other_words, repr_function="__repr__", params=None):
    """
    Sorts a list of objects by their relatedness to a given plain text.

    The object's representation function can also be defined as well as specific arguments which have to be used with
    that function.

    :param word: Word to compare to.
    :type word: str
    :param other_words: list of objects to sort.
    :type other_words: list
    :param repr_function: Name of the representation function of the object to be used to obtain on objects string representation. Defaults to __repr__
    :type repr_function: str
    :param params: Dict of keyword arguments to be used with the representation function. Defaults to None.
    :type params: dict
    :return: List of objects sorted by their semantic relatedness.
    :rtype: list
    """
    params = params if params else {}
    words_with_ids = [
        {
            "_id": id,
            "name": getattr(name, repr_function)(**params)
        }
        for id, name in enumerate(other_words)]
    result = sort_by_relatedness_with_id(word, words_with_ids)

    return [other_words[int(id)] for id in result]
