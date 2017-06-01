class PhoneAPI:
    def call(self, number, text):
        """
        Blablablablabla.
        
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