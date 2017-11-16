from aide_core.apis import indra

def get_intent(plain_text):
    """
    Examplary function to get the intent from a given plain text.

    Intent can be "joke", "weather" or "location"

    Obviosly just for the sake of demonstration, as it is not very sophisticated.

    :param plain_text: Text to get the intent from.
    :type plain_text: str
    :return: Intent of message
    :rtype: str
    """
    intent = ["location position", "joke", "weather"]
    if "where" in plain_text:
        return "location"
    result = indra.sort_by_relatedness(plain_text, intent)
    return result[0].split(" ", 1)[0]