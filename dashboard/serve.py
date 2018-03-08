#!/usr/bin/env python3

from aiohttp import web
import aiohttp_jinja2
import jinja2

app = web.Application()
aiohttp_jinja2.setup(
    app, loader=jinja2.FileSystemLoader('views')
)

async def index(request):
    return aiohttp_jinja2.render_template('index.html', request, {})

app.router.add_static('/dist', 'dist')
app.router.add_get('/', index)
app.router.add_get('/{name}', index) # anything else is frontend routed
web.run_app(app)
