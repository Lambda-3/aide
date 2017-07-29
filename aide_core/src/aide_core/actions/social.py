from apis import joke

from aide_core.actions import say


def tell_joke():
    say(joke.get_random_joke())