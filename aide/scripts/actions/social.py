from speech import say
from apis import joke

def tell_joke():
    say(joke.get_random_joke())

def tell_location():
    say(location.get_location_human_readable())