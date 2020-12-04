import imaplib, email, time, os

def receive():
    # setup email login info here
    genios_email = "chattygenios@gmail.com"
    target_email = genios_email

    # login to gmail
    server = imaplib.IMAP4_SSL('imap.gmail.com')
    server.login(genios_email, 'midcamidca')
    print ("Connected to inbox!")

    # periodically check for new emails (if setting up when emails have already been sent, change last_email_id to the last_email_id of the previous instantiation)
    last_email_id = -1
    while True:
      server.select("inbox")
      result, data = server.search(None, "ALL")

      ids = data[0]
      id_list = ids.split()
      latest_email_id = id_list[-1]
      if latest_email_id != last_email_id:
        last_email_id = latest_email_id

        # get/check for email attachment
        result, data = server.fetch(latest_email_id, "(RFC822)")
        raw_email = data[0][1]
        msg = email.message_from_string(raw_email)

        for part in msg.walk():
          if part.get_content_maintype() == 'multipart': continue
          if part.get('Content-Disposition') is None: continue
          fileName = part.get_filename()

          if fileName is not None:
            filePath = os.getcwd()+"/communication/"+fileName
            fp = open(filePath, 'wb')
            fp.write(part.get_payload(decode=True))
            fp.close()
            print ("Downloaded attachment!")

        time.sleep(1)
