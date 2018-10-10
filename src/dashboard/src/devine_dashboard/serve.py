#!/usr/bin/env python3
'''Python server of the DEVINE's dashboard'''

from aiohttp import web
import aiohttp_jinja2
import jinja2
import os
from devine_config import gettopics

CURRENT_DIRECTORY = os.path.dirname(os.path.realpath(__file__))

app = web.Application()
aiohttp_jinja2.setup(
    app, loader=jinja2.FileSystemLoader(os.path.join(CURRENT_DIRECTORY, 'html'))
)

async def index(request):
    '''Default http request handler'''
    path = request.match_info.get('path', "index")
    return aiohttp_jinja2.render_template('index.html', request, {'path': path})

async def topics_api(_):
    '''Api access to the ROS topics of DEVINE'''
    return web.json_response(gettopics())

app.router.add_static('/css', os.path.join(CURRENT_DIRECTORY, 'css'))
app.router.add_static('/dist', os.path.join(CURRENT_DIRECTORY, '../../dist'))
app.router.add_get('/topics', topics_api)
app.router.add_get('/', index)
app.router.add_get('/{path}', index) # anything else is frontend routed
web.run_app(app)
