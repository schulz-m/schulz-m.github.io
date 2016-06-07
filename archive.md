---
layout: page
title: Archive
permalink: /archive/
---

<div>
{% for post in site.posts %}
	{% capture this_year %}{{ post.date | date: "%Y" }}{% endcapture %}

	{% unless year == this_year %}
		{% assign year = this_year %}
		{% unless post == site.posts.first %}
		{% endunless %}
		<h2 id="{{ year }}">{{ year }}</h2>
	{% endunless %}

	<time datetime="{{ post.date | date:"%Y-%m-%d" }}">
	{{ post.date | date:"%Y-%m-%d" }}
	</time>

	<a href="{{ site.baseurl }}{{ post.url }}">{{ post.title | capitalize }}</a><br />
{% endfor %}
</div>