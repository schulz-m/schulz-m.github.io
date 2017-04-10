### Automate with Selenium and Python

This post was inspired by the excelent introduction of [Irwin Kwan](https://irwinkwan.com/2013/04/05/automating-the-web-with-selenium-complete-tasks-automatically-and-write-test-cases/) on his personal website.

The selenium python package is given on [pypi](https://pypi.python.org/pypi/selenium/3.3.0) where one can also find the installation instructions - don't forget to install the browser specific drivers as indicated on the site. As an example the driver installation for firefox on Ubuntu is like this:

* Go on the geckodriver [website](https://github.com/mozilla/geckodriver/releases) and download the latest version, e.g. `geckodriver-v0.15.0-linux64.tar.gz`.
* Install the driver like this:
    - `tar -xvf geckodriver-v0.15.0-linux64.tar`
    - `sudo mv geckodriver /usr/local/bin/`

Check if the installation worked by opening Firefox as given in the Example 0.

Now for this introduction we are using the Selenium IDE to directly generate the python code that does what we want [plugin link](https://addons.mozilla.org/en-US/firefox/addon/selenium-ide/). The plugin can be opened via Tools->Selenium IDE. Now we are ready to go!

The Selenium IDE by itself will show you this kinda stuff and is automatically recording, but we want to actually use the driver so what we do is to open up the following.


This is a manual solver... lol
Now we could try the captcha solver:
https://pypi.python.org/pypi/captcha-solver

can simply be installed by pip again - `sudo pip install captcha-solver`


So now we are using Tesseract OCR https://pypi.python.org/pypi/tesserocr
actually installing this https://pypi.python.org/pypi/pytesseract/0.1

--- again simply sudo pip install

A good post on using openCV for the captcha http://www.robindavid.fr/opencv-tutorial/cracking-basic-captchas-with-opencv.html

Use image_slicer https://pypi.python.org/pypi/image_slicer

and then use option - psm -10 to assume single characters.

Explanation how to automatically read mails here: http://codehandbook.org/how-to-read-email-from-gmail-using-python/

Didn't work for me so I went for this pypi library: https://pypi.python.org/pypi/pygmail/0.0.5.2

pretty cool functionalities:
https://github.com/charlierguo/gmail



So for this to work with gmail, it depends on the security settings you have - like https://www.google.com/settings/security/lesssecureapps
















