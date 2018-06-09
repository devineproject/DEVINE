#!/usr/bin/env python3

from aiohttp import web
import aiohttp_jinja2
import jinja2

app = web.Application()
aiohttp_jinja2.setup(
    app, loader=jinja2.FileSystemLoader('views')
)

async def index(request):
    path = request.match_info.get('path', "index")
    return aiohttp_jinja2.render_template('index.html', request, {'path': path})

app.router.add_static('/assets', 'assets')
app.router.add_static('/dist', 'dist')
app.router.add_get('/', index)
app.router.add_get('/{path}', index) # anything else is frontend routed
web.run_app(app)
