from aide_core.apis import joke

from aide_core.actions.speech import say


def tell_joke():
    say(joke.get_random_joke())