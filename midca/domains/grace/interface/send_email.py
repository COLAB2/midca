import smtplib, time
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase 
from email import encoders 

def send(command=None, filename = "data.json"):
    # provide file to attach
    attach_path = filename#"test.txt"

    # provide emails to send with
    genios_email = "metamidca@gmail.com"
    target_email = "chattygenios@gmail.com"

    # setup the gmail server and login
    server = smtplib.SMTP_SSL('smtp.gmail.com', 465)
    server.ehlo()
    server.login(genios_email, 'midcamidca')

    # generate the message attachment
    msg = MIMEMultipart()
    msg['From'] = genios_email
    msg['To'] = target_email
    msg['Subject'] = command
    attachment = open(attach_path, "rb")
    p = MIMEBase('application', 'octet-stream')
    p.set_payload((attachment).read())
    encoders.encode_base64(p)
    p.add_header('Content-Disposition', "attachment; filename= %s" % attach_path)

    msg.attach(p)

    # send an email to yourself
    server.sendmail(genios_email, target_email, msg.as_string())

    # shutdown the connection
    server.close()
