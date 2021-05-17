#! /usr/bin/env python3

def plan(domainfile, statefile, search_key, search_method):
    from midca.modules._plan.fastdown.driver.main import main
    main(domainfile, statefile, search_key, search_method)
    #main()
