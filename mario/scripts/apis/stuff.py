class PhoneAPI:
    def call(self, number, text):
        """
        
        :param number: 
        :param text: 
        :return: 
        """
        print(20 * "=")
        print("Calling {}...".format(number))
        print("...connection established, saying text:")
        for row in text.splitlines():
            print("\t" + row)
        print("Hanging up.")
        print(20 * "=")

    def send_message(self, number, message):
        print(20 * "=")
        print("Sending message to {} with content \"{}\".".format(number, message))
        print(20 * "=")


class MoveAPI:
    def move(self, coordinates):
        print(20 * "=")
        print("Moving to coordinates {}.".format(coordinates))

        print(20 * "=")

    def get_nearest_corner(self):
        return (1.3, 3.7)
