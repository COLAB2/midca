from subprocess import Popen

def jshop(tasks):

    p = Popen("j.bat", cwd=r"C:/Users/Zohreh/Google Drive/SHOP/Practice-copy/bin/")
    stdout, stderr = p.communicate()
    
    filename = "C:/Users/Zohreh/Google Drive/SHOP/Practice-copy/bin/output.txt"
    with open(filename) as f:
        content = f.readlines()
    
    for line in content:
        if(line.startswith(" ( (!")):
            plan = line
            break
    
    if(plan):   
        Jshop_plan = parse(plan)
    
    return Jshop_plan

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
        if(elm[0] == '!'):
            print elm
            plan.append(elm)
    
    return plan        
            
if __name__ == "__main__": 
    jshop("tasks")
            
            
            