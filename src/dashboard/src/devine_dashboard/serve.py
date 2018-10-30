#!/usr/bin/env python3
""" Python server of the DEVINE's dashboard """

import os
from aiohttp import web
import aiohttp_jinja2
import jinja2
from devine_config import gettopics

CURRENT_DIRECTORY = os.path.dirname(os.path.realpath(__file__))

APP = web.Application()
aiohttp_jinja2.setup(
    APP, loader=jinja2.FileSystemLoader(os.path.join(CURRENT_DIRECTORY, 'html'))
)


async def index(request):
    """ Default http request handler """
    path = request.match_info.get('path', 'index')
    return aiohttp_jinja2.render_template('index.html', request, {'path': path})


async def topics_api(_):
    """ Api access to the ROS topics of DEVINE """
    return web.json_response(gettopics())

APP.router.add_static('/css', os.path.join(CURRENT_DIRECTORY, 'css'))
APP.router.add_static('/dist', os.path.join(CURRENT_DIRECTORY, '../../dist'))
APP.router.add_get('/topics', topics_api)
APP.router.add_get('/', index)
APP.router.add_get('/{path}', index)  # anything else is frontend routed
web.run_app(APP)
