import json
import requests

from util import to_space

_data = {'corpus'       : 'wiki-2014',
         'model'        : 'W2V',
         'language'     : 'EN',
         'scoreFunction': 'COSINE'}

_headers = {
    'content-type': "application/json"
}

_endpoint = "http://localhost:8916/relatedness"


def get_semantic_relatedness(pairs):
    _data['pairs'] = pairs
    response = requests.request("POST", _endpoint, data=json.dumps(_data), headers=_headers)

    response.raise_for_status()

    response = response.json()['pairs']
    return response


def sort_by_relatedness_with_id(word, words_with_ids, id_name="_id", format="{name}"):
    pairs = [{"t1": to_space(word),
              "t2": to_space(("{id} | " + format).format(
                  id=words_dict[id_name], **{k: v for k, v in words_dict.iteritems() if not k == id_name}))}
             for words_dict in words_with_ids]
    result = sorted(get_semantic_relatedness(pairs), reverse=True)
    return [entry["t2"].split(" | ", 1)[0] for entry in result]


def sort_by_relatedness(word, other_words):
    ids = range(len(other_words))
    words_with_ids = [{"_id": id, "name": name} for id, name in zip(ids, other_words)]
    result = sort_by_relatedness_with_id(word, words_with_ids)
    return [other_words[int(id)] for id in result]
