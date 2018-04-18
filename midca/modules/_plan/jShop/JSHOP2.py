import inspect, os
import subprocess
import sys
from subprocess import Popen, PIPE, STDOUT

###Make sure that Java is installed fully on your computer.
###set the CLASSPATH
###environment variable to include (replacing JSHOP2_DIRECTORY with the directory
###where JSHOP2 is):
### in Windows:
###JSHOP2_DIRECTORY\bin\antlr.jar;JSHOP2_DIRECTORY\bin\JSHOP2.jar;.

###- in UNIX:
###JSHOP2_DIRECTORY/bin/antlr.jar:JSHOP2_DIRECTORY/bin/JSHOP2.jar:.

###If you have problem running the current jar file you can compile it again and put the jar file here, 

def compile_java(java_file):
    subprocess.check_call(['javac', java_file])

def execute_java(java_file, stdin):
    java_class,ext = os.path.splitext(java_file)
    cmd = ['java', java_class]
    proc = subprocess.Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=STDOUT)
    stdout,stderr = proc.communicate(stdin)
    print ('This was "' + stdout + '"')


def jshop(tasks, DOMAIN_FIILE,STATE_FILE):
    
    thisDir =  os.path.dirname(os.path.realpath(__file__))
    
#     thisDir = "C:/Users/Zohreh/git/MIDCA/modules/_plan/jShop/"
    MIDCA_ROOT = thisDir + "/../../../"
    
#     DOMAIN_FIILE = MIDCA_ROOT + "domains/jshop_domains/logistics/logistics"
#     #DOMAIN_FIILE = JSHOP_ROOT + "domains/jshop_domains/blocks_world/blocksworld.shp"
#     STATE_FILE = MIDCA_ROOT + "domains/jshop_domains/logistics/problem"
#     print thisDir
    cwd = os.getcwd()
    
#     print cwd
    os.chdir(thisDir)
    
    
    args = ['java', 
        r"-classpath", 
        r".;./JSHOP2.jar;./antlr.jar", 
        r"JSHOP2.InternalDomain", 
        DOMAIN_FIILE 
       ] 
    proc = subprocess.Popen(args, stdout=subprocess.PIPE, cwd=thisDir) 
    proc.communicate() 

    args = ['java', 
        r"-cp", 
        r".;./JSHOP2.jar;./antlr.jar", 
        r"JSHOP2.InternalDomain",
        "-r", 
        STATE_FILE
       ] 
    proc = subprocess.Popen(args, stdout=subprocess.PIPE) 
    proc.communicate() 
    
    
    args = ['javac',
            r"-classpath", 
            r".;JSHOP2.jar;antlr.jar",  
            r"problem.java", 
       ] 
    proc = subprocess.Popen(args, stdout=subprocess.PIPE) 
    proc.communicate() 
    
    args = ['java',
            r"-classpath", 
            r".;JSHOP2.jar;antlr.jar",  
            r"problem", 
       ] 
    proc = subprocess.Popen(args, stdout=subprocess.PIPE) 
    p,l = proc.communicate()
    
    plans = p.split("\r\n")
    midcaplan = parse(p)

     
    
     
    return midcaplan

def parenthetic_contents(string):
    """Generate parenthesized contents in string as pairs (level, contents)."""
    stack = []
    for i, c in enumerate(string):
        if c == '(':
            stack.append(i)
        elif c == ')' and stack:
            start = stack.pop()
            yield (string[start + 1: i])

def parse(str):
    elements  = list(parenthetic_contents(str))
    plan = []
    for elm in elements:
        if(elm[0] == '!' and '(' not in elm):
            elm = elm[1:]
#             print elm
            action_list = elm.strip().split(' ')
            plan.append(action_list)
    
    return plan        
            
if __name__ == "__main__": 
    thisDir =  os.path.dirname(os.path.realpath(__file__))
    MIDCA_ROOT = thisDir + "/../../../"
    
    DOMAIN_FIILE = MIDCA_ROOT + "domains/jshop_domains/logistics/logistics"
#     #DOMAIN_FIILE = JSHOP_ROOT + "domains/jshop_domains/blocks_world/blocksworld.shp"
    STATE_FILE = MIDCA_ROOT + "domains/jshop_domains/logistics/problem"
    jshop("tasks", DOMAIN_FIILE, STATE_FILE)
            
            
            